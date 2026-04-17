/***************************************************************************
 *   This library is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU Lesser General Public License version   *
 *   2.1 as published by the Free Software Foundation.                     *
 *                                                                         *
 *   This library is distributed in the hope that it will be useful, but   *
 *   WITHOUT ANY WARRANTY; without even the implied warranty of            *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU     *
 *   Lesser General Public License for more details.                       *
 *                                                                         *
 *   You should have received a copy of the GNU Lesser General Public      *
 *   License along with this library; if not, write to the Free Software   *
 *   Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA         *
 *   02110-1301  USA                                                       *
 *                                                                         *
 *   Alternatively, this file is available under the Mozilla Public        *
 *   License Version 1.1.  You may obtain a copy of the License at         *
 *   http://www.mozilla.org/MPL/                                           *
 ***************************************************************************/

#include "ebmlmksegment.h"
#include "ebmlutils.h"
#include "matroskafile.h"
#include "matroskatag.h"
#include "matroskaattachments.h"
#include "matroskachapters.h"
#include "matroskacues.h"
#include "matroskaseekhead.h"
#include "matroskasegment.h"

using namespace TagLib;

namespace {

template <EBML::Element::Id Id, typename ElementType>
std::unique_ptr<ElementType> readElementAt(File &file,
                                           offset_t offset,
                                           offset_t maxOffset)
{
  if(offset < 0 || offset >= maxOffset) {
    return nullptr;
  }

  file.seek(offset);
  auto element = EBML::Element::factory(file);
  if(!element || element->getId() != Id) {
    return nullptr;
  }

  auto typed = EBML::element_cast<Id>(std::move(element));
  if(!typed || !typed->read(file)) {
    return nullptr;
  }
  return typed;
}

} // namespace

EBML::MkSegment::MkSegment(int sizeLength, offset_t dataSize, offset_t offset):
  MasterElement(Id::MkSegment, sizeLength, dataSize, offset)
{
}

EBML::MkSegment::MkSegment(Id, int sizeLength, offset_t dataSize, offset_t offset):
  MasterElement(Id::MkSegment, sizeLength, dataSize, offset)
{
}

EBML::MkSegment::~MkSegment() = default;

offset_t EBML::MkSegment::segmentDataOffset() const
{
  return offset + idSize(id) + sizeLength;
}

bool EBML::MkSegment::read(File &file)
{
  return readLimited(file, dataSize);
}

bool EBML::MkSegment::readLimited(File& file, offset_t scanLimit, bool findChapters)
{
  const offset_t filePos = file.tell();
  const offset_t maxOffset = filePos + dataSize;
  const offset_t maxScanOffset = filePos + std::min(scanLimit, dataSize);
  const bool isFastScan = scanLimit < dataSize;
  std::unique_ptr<Element> element;
  while((element = findNextElement(file, maxScanOffset))) {
    if(const Id id = element->getId(); id == Id::MkSeekHead) {
      seekHead = element_cast<Id::MkSeekHead>(std::move(element));
      if(!seekHead->read(file))
        return false;
      // We have a seek head, let's use it for faster access to the other elements
      if(const auto elementAfterSeekHead = findNextElement(file, maxScanOffset);
         elementAfterSeekHead && elementAfterSeekHead->getId() == Id::VoidElement)
        seekHead->setPadding(elementAfterSeekHead->getSize());
      const offset_t segDataOffset = segmentDataOffset();
      const auto matroskaSeekHead = parseSeekHead();
      for(const auto &[idValue, relativeOffset] : matroskaSeekHead->entryList()) {
        const offset_t absoluteOffset = segDataOffset + relativeOffset;
        switch(static_cast<Id>(idValue)) {
        case Id::MkCues:
          // Skip Cues in fast scan mode — they are only needed for Accurate validation
          // and can be tens of MB on large files, causing severe network slowdowns
          if(!isFastScan) {
            if(!((cues = readElementAt<Id::MkCues, MkCues>(
              file, absoluteOffset, maxOffset))))
              return false;
          }
          break;
        case Id::MkInfo:
          if(!((info = readElementAt<Id::MkInfo, MkInfo>(
            file, absoluteOffset, maxOffset))))
            return false;
          break;
        case Id::MkTracks:
          if(!((tracks = readElementAt<Id::MkTracks, MkTracks>(
            file, absoluteOffset, maxOffset))))
            return false;
          break;
        case Id::MkTags:
          if(!((tags = readElementAt<Id::MkTags, MkTags>(
            file, absoluteOffset, maxOffset))))
            return false;
          break;
        case Id::MkAttachments:
          if(!((attachments = readElementAt<Id::MkAttachments, MkAttachments>(
            file, absoluteOffset, maxOffset))))
            return false;
          break;
        case Id::MkChapters:
          if(!((chapters = readElementAt<Id::MkChapters, MkChapters>(
            file, absoluteOffset, maxOffset))))
            return false;
          break;
        default:
          break;
        }
      }
      // If all essential elements were found via the SeekHead, we're done.
      if(chapters && tags && info)
        return true;
      // Only do the expensive fallback scan when chapters are specifically needed
      // (read path). The save path doesn't need chapters and should skip this to
      // avoid slow network reads.
      if(tags && info && !findChapters)
        return true;
      // Some older MKV files don't index all elements in the SeekHead (e.g. Chapters).
      // Do a targeted scan with a larger limit to find missing elements without
      // reading the entire file (which would be extremely slow over network drives).
      static constexpr offset_t FALLBACK_SCAN_LIMIT = 10 * 1024 * 1024; // 10 MiB
      const offset_t fallbackMaxOffset = std::min(filePos + FALLBACK_SCAN_LIMIT, maxOffset);
      file.seek(filePos);
      while((element = findNextElement(file, fallbackMaxOffset))) {
        const Id eid = element->getId();
        const offset_t elementEndOffset = file.tell() + element->getDataSize();
        if(!chapters && eid == Id::MkChapters) {
          chapters = element_cast<Id::MkChapters>(std::move(element));
          if(!chapters->read(file)) {
            // Partial read is OK for chapters in older files with padding/unknown elements.
            // Seek past the element and keep whatever was successfully parsed.
            file.seek(elementEndOffset);
          }
        }
        else if(!tags && eid == Id::MkTags) {
          tags = element_cast<Id::MkTags>(std::move(element));
          if(!tags->read(file))
            file.seek(elementEndOffset);
        }
        else if(!info && eid == Id::MkInfo) {
          info = element_cast<Id::MkInfo>(std::move(element));
          if(!info->read(file))
            file.seek(elementEndOffset);
        }
        else if(!attachments && eid == Id::MkAttachments) {
          attachments = element_cast<Id::MkAttachments>(std::move(element));
          if(!attachments->read(file))
            file.seek(elementEndOffset);
        }
        else {
          element->skipData(file);
        }
        // Stop early if we found everything
        if(chapters && tags && info)
          return true;
      }
      return true;
    }
    else if(id == Id::MkCues) {
      // Skip Cues in fast scan mode
      if(isFastScan) {
        element->skipData(file);
      }
      else {
        cues = element_cast<Id::MkCues>(std::move(element));
        if(!cues->read(file))
          return false;
      }
    }
    else if(id == Id::MkInfo) {
      info = element_cast<Id::MkInfo>(std::move(element));
      if(!info->read(file))
        return false;
    }
    else if(id == Id::MkTracks) {
      tracks = element_cast<Id::MkTracks>(std::move(element));
      if(!tracks->read(file))
        return false;
    }
    else if(id == Id::MkTags) {
      tags = element_cast<Id::MkTags>(std::move(element));
      if(!tags->read(file))
        return false;
    }
    else if(id == Id::MkAttachments) {
      attachments = element_cast<Id::MkAttachments>(std::move(element));
      if(!attachments->read(file))
        return false;
    }
    else if(id == Id::MkChapters) {
      chapters = element_cast<Id::MkChapters>(std::move(element));
      if(!chapters->read(file))
        return false;
    }
    else {
      element->skipData(file);
    }
  }
  return true;
}

std::unique_ptr<Matroska::Tag> EBML::MkSegment::parseTag() const
{
  return tags ? tags->parse() : nullptr;
}

std::unique_ptr<Matroska::Attachments> EBML::MkSegment::parseAttachments() const
{
  return attachments ? attachments->parse() : nullptr;
}

std::unique_ptr<Matroska::Chapters> EBML::MkSegment::parseChapters() const
{
  return chapters ? chapters->parse() : nullptr;
}

std::unique_ptr<Matroska::SeekHead> EBML::MkSegment::parseSeekHead() const
{
  return seekHead ? seekHead->parse(segmentDataOffset()) : nullptr;
}

std::unique_ptr<Matroska::Cues> EBML::MkSegment::parseCues() const
{
  return cues ? cues->parse(segmentDataOffset()) : nullptr;
}

std::unique_ptr<Matroska::Segment> EBML::MkSegment::parseSegment() const
{
  return std::make_unique<Matroska::Segment>(sizeLength, dataSize, offset + idSize(id));
}

void EBML::MkSegment::parseInfo(Matroska::Properties *properties) const
{
  if(info) {
    info->parse(properties);
  }
}

void EBML::MkSegment::parseTracks(Matroska::Properties *properties) const
{
  if(tracks) {
    tracks->parse(properties);
  }
}
