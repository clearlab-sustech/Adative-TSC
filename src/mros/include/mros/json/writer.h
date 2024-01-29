#ifndef MROS_JOSN_WRITER_H
#define MROS_JOSN_WRITER_H

#include "mros/json/elements.h"
#include "mros/json/visitor.h"
namespace mros
{
namespace json
{

class Writer : private ConstVisitor
{
public:
   static void Write(const Object& object, std::ostream& ostr, bool pretty = false);
   static void Write(const Array& array, std::ostream& ostr, bool pretty = true);
   static void Write(const String& string, std::ostream& ostr);
   static void Write(const Number& number, std::ostream& ostr);
   static void Write(const Boolean& boolean, std::ostream& ostr);
   static void Write(const Null& null, std::ostream& ostr);
   static void Write(const UnknownElement& elementRoot, std::ostream& ostr);

private:
   Writer(std::ostream& ostr, bool pretty = true);

   template <typename ElementTypeT>
   static void Write_i(const ElementTypeT& element, std::ostream& ostr, bool pretty = true);

   void Write_i(const Object& object);
   void Write_i(const Array& array);
   void Write_i(const String& string);
   void Write_i(const Number& number);
   void Write_i(const Boolean& boolean);
   void Write_i(const Null& null);
   void Write_i(const UnknownElement& unknown);

   virtual void Visit(const Array& array);
   virtual void Visit(const Object& object);
   virtual void Visit(const Number& number);
   virtual void Visit(const String& string);
   virtual void Visit(const Boolean& boolean);
   virtual void Visit(const Null& null);

   std::ostream& m_ostr;
   bool m_pretty;
   int m_nTabDepth;
};

}
} // End namespace


#include "mros/json/writer.inl"

#endif
